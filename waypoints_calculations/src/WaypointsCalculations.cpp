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
#include <algorithm>

using namespace std;


class WaypointsCalculations : public rclcpp::Node
{
private:
    // Variables from parameters
    double lookahead_min = 0.0; // [m]
    double lookahead_max = 0.0; // [m]
    double mps_alpha = 0.0; // [m/s]
    double mps_beta = 0.0; // [m/s]
    // Variables
    double current_yaw_ = 0.0; // [rad]
    double current_x_ = 0.0; // [m]
    double current_y_ = 0.0; // [m]
    double current_velocity_ = 0; // [m/s]
    // Waypoints variables
    int closest_waypoint = 0;
    double setVelocity = 0.0;
    float average_distance = 0.0, maximum_distance = 0.0;
    bool waypoints_in_loop = false;
    int average_distance_count = 0;
    size_t target_waypoint = 0;

    vector<Eigen::VectorXd> waypoints; // [x, y, z, yaw]
    vector<float> velocities;
    // Computations functions
    void pub_callback();
    double LookAheadDistance();
    void waypointsComputation();
    double getDistance(Eigen::VectorXd point1, Eigen::VectorXd point2);
    void setVelocityToSend();
    double getDistanceFromOdom(Eigen::VectorXd wapointPoint);
    
    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void waypoints_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void velocities_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void marketTargetWaypoint();
    // Subcribers, timers & publishers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr velocities_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_waypoint_pub_;
public:
    WaypointsCalculations(/* args */);
    ~WaypointsCalculations();
};

WaypointsCalculations::WaypointsCalculations(/* args */) : Node("waypoints_calculations")
{

    this->declare_parameter("lookahead_min", double(0.0));
    this->declare_parameter("lookahead_max", double(0.0));
    this->declare_parameter("mps_alpha", double(0.0));
    this->declare_parameter("mps_beta", double(0.0));

    this->get_parameter("lookahead_min", lookahead_min);
    this->get_parameter("lookahead_max", lookahead_max);
    this->get_parameter("mps_alpha", mps_alpha);
    this->get_parameter("mps_beta", mps_beta);

    target_waypoint_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("target_waypoint_marker", 10);

    waypoints_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
        "waypoints_loaded", 10,
        std::bind(&WaypointsCalculations::waypoints_callback, this, std::placeholders::_1)
    );

    velocities_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "waypointarray_speeds", 10,
        std::bind(&WaypointsCalculations::velocities_callback, this, std::placeholders::_1)
    );

    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&WaypointsCalculations::odom_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&WaypointsCalculations::pub_callback, this));
    RCLCPP_INFO(this->get_logger(), "waypoints_calculations initialized");

}

WaypointsCalculations::~WaypointsCalculations()
{

}

// TIMER CALLBACK
void WaypointsCalculations::pub_callback()
{
    waypointsComputation();
    setVelocityToSend();
}

// ODOM CALLBACK    
void WaypointsCalculations::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
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

void WaypointsCalculations::velocities_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    velocities.clear();
    velocities.insert(velocities.end(), msg->data.begin(), msg->data.end());
}

void WaypointsCalculations::waypoints_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
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

void WaypointsCalculations::setVelocityToSend(){
    if(closest_waypoint == 0){
        setVelocity = velocities[1];
    }else{
        setVelocity = velocities[closest_waypoint];
    }
}

double WaypointsCalculations::getDistance(Eigen::VectorXd point1, Eigen::VectorXd point2){
    double distance = sqrt(pow(point1(0) - point2(0), 2) + pow(point1(1) - point2(1), 2));
    return distance;
}

double WaypointsCalculations::LookAheadDistance(){
    double clamped_speed = std::clamp(current_velocity_, mps_alpha, mps_beta);
    return (lookahead_max - lookahead_min) / (mps_beta - mps_alpha) * (clamped_speed - mps_alpha) + lookahead_min;
}

double WaypointsCalculations::getDistanceFromOdom(Eigen::VectorXd wapointPoint){
    double x1 = wapointPoint(0);
    double y1 = wapointPoint(1);
    double x2 = current_x_;
    double y2 = current_y_;
    double distance = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
    return distance;
}

void WaypointsCalculations::waypointsComputation(){
    constexpr size_t first_wp = 0;                        
    const size_t last_wp = waypoints.size() - 1; 
    constexpr double max_distance_threshold = 10.0;
    constexpr int search_offset_back = 5;
    constexpr int search_offset_forward = 15;

    // loop closure is true, when the first and last waypoint are closer than 4.0 meters
    double distance = getDistance(waypoints[first_wp], waypoints[last_wp]);
    waypoints_in_loop = distance < 4.0;

    // print waypoints_in_loop
    // RCLCPP_INFO(this->get_logger(), "waypoints_in_loop: %d", waypoints_in_loop);

    int search_start = std::max(static_cast<int>(closest_waypoint) - search_offset_back, static_cast<int>(first_wp));
    int search_end = std::min(static_cast<int>(closest_waypoint) + search_offset_forward, static_cast<int>(last_wp));

    double smallest_curr_distance = std::numeric_limits<double>::max();
    for (int i = search_start; i <= search_end; i++)
    {
        double curr_distance = getDistanceFromOdom(waypoints[i]);
        if (smallest_curr_distance > curr_distance)
        {
            closest_waypoint = i;
            smallest_curr_distance = curr_distance;
        }
    }

    if (average_distance_count == 0) {
        average_distance = smallest_curr_distance;
    } else {
        average_distance = (average_distance * average_distance_count + smallest_curr_distance) / (average_distance_count + 1);
    }
    average_distance_count++;

    // Updating maximum_distance
    if (maximum_distance < smallest_curr_distance && smallest_curr_distance < max_distance_threshold) {
        maximum_distance = smallest_curr_distance;
    }

    double lookahead_actual = LookAheadDistance(); 

    for (int i = closest_waypoint; i <= static_cast<int>(last_wp); i++)
    {
        double curr_distance = getDistanceFromOdom(waypoints[i]);
        if (curr_distance > lookahead_actual && curr_distance < lookahead_actual + 8.0 )
        {
            target_waypoint = i;            
            break;
        }
        // set if is loop true and the current waypoint is the last waypoint set the target waypoint to the first waypoint
        if (waypoints_in_loop && i == static_cast<int>(last_wp)) {
            target_waypoint = first_wp;
        }else{
            target_waypoint = last_wp;
            // finishedpath = true;
        }
        // target_waypoint = last_wp;
    }
}

void WaypointsCalculations::marketTargetWaypoint(){
    // Publish target waypoint marker

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odom";  // Adjust this to your frame
    marker.header.stamp = this->now();
    marker.ns = "target_waypoint";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = waypoints[target_waypoint](0);
    marker.pose.position.y = waypoints[target_waypoint](1);
    marker.pose.position.z = waypoints[target_waypoint](2);
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    
    target_waypoint_pub_->publish(marker);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointsCalculations>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
