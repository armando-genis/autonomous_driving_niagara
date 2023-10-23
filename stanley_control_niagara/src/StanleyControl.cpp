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



class StanleyControl : public rclcpp::Node
{
private:
    /* data */

    double max_steer = 0.0; // [rad]
    double lookahead_min = 0.0; // [m]
    double lookahead_max = 0.0; // [m]
    double mps_alpha = 0.0; // [m/s]
    double mps_beta = 0.0; // [m/s]
    double current_yaw_ = 0.0; // [rad]
    double current_x_ = 0.0; // [m]
    double current_y_ = 0.0; // [m]
    double target_idx = 0.0;

    double L = 0.04; // [m] Wheel base of vehicle
    double error_front_axle = 0.0;
    double wheelbase = 2.789;
    double heading_err_rate = 0.05; 
    double stering_angle = 0.0;
    float pursuit_rate = 0.9; 
    double K = 0.5;  // control gain

    // size_t closest_waypoint = 0;
    bool waypoints_in_loop = false;
    int average_distance_count = 0;
    size_t target_waypoint = 0;
    bool finishedpath = false;

    vector<Eigen::VectorXd> waypoints; // [x, y, z, yaw, velocity]
    vector<Eigen::VectorXd> waypointsCubeLines;
    vector<float> velocities;

    int closest_waypoint = 0;
    float average_distance = 0.0, maximum_distance = 0.0;
    double current_velocity_ = 0;
    double setVelocity = 0.0;


    nav_msgs::msg::Odometry current_pose_;
    geometry_msgs::msg::Pose current_pose_from_odom_;
    float cross_track_err_rate = 0.05;

    // Computations functions
    void pub_callback();
    double GetNormaliceAngle(double angle);
    void waypointsComputation();
    double LookAheadDistance(double velocity);
    void setVelocityToSend();
    void marketTargetWaypoint();
    
    void computeCrossTrackError();
    void computeSteeringAngle();

    // Callbacks
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void waypointCubeLinesCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void waypoints_callback(const visualization_msgs::msg::MarkerArray::SharedPtr msg);
    void velocities_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    // utils functions
    double getDistance(Eigen::VectorXd point1, Eigen::VectorXd point2);

    // Subcribers, timers & publishers
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr velocities_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_subscriber_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr target_waypoint_pub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr motor_pub_;


public:
    StanleyControl(/* args */);
    ~StanleyControl();
};

StanleyControl::StanleyControl(/* args */) : Node("stanley_control_niagara_node")
{
    this->declare_parameter("wheelbase", double(0.0));
    this->declare_parameter("max_steer", double(0.0));
    this->declare_parameter("lookahead_min", double(0.0));
    this->declare_parameter("lookahead_max", double(0.0));
    this->declare_parameter("mps_alpha", double(0.0));
    this->declare_parameter("mps_beta", double(0.0));

    this->get_parameter("wheelbase", wheelbase);
    this->get_parameter("max_steer", max_steer);
    this->get_parameter("lookahead_min", lookahead_min);
    this->get_parameter("lookahead_max", lookahead_max);
    this->get_parameter("mps_alpha", mps_alpha);
    this->get_parameter("mps_beta", mps_beta);

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
        std::bind(&StanleyControl::waypointCubeLinesCallback, this, std::placeholders::_1)
    );
    
    target_waypoint_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("target_waypoint_marker", 10);
    motor_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/ackermann_cmd", 10);


    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&StanleyControl::odom_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&StanleyControl::pub_callback, this));
    RCLCPP_INFO(this->get_logger(), "stanley_control_niagara_node initialized");

}

StanleyControl::~StanleyControl()
{
}

void StanleyControl::pub_callback()
{
    // RCLCPP_INFO(this->get_logger(), "stanley_control_niagara_node callback");
    // print parameters
    // RCLCPP_INFO(this->get_logger(), "wheelbase: %f", wheelbase);
    // RCLCPP_INFO(this->get_logger(), "max_steer: %f", max_steer);

    // print fist point of the waypoints
    // RCLCPP_INFO(this->get_logger(), "waypoints[0]: %f, %f, %f, %f, %f" , waypoints[0](0), waypoints[0](1), waypoints[0](2), waypoints[0](3), velocities[0]);
    // sent the size of waypoitns: 
    // RCLCPP_INFO(this->get_logger(), "waypoints.size(): %d", waypointsCubeLines.size());
    waypointsComputation();
    setVelocityToSend();
    marketTargetWaypoint();
    computeCrossTrackError();
    computeSteeringAngle();


    auto ackermann_cmd_msg = ackermann_msgs::msg::AckermannDriveStamped();
    // Assuming speed_ and steering_angle_ are member variables that hold your desired speed and steering angle
    ackermann_cmd_msg.drive.speed = setVelocity;  
    ackermann_cmd_msg.drive.steering_angle = stering_angle;
    // Publish the message
    motor_pub_->publish(ackermann_cmd_msg);

}

void StanleyControl::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_pose_ = *msg;
    current_x_ = current_pose_.pose.pose.position.x;
    current_y_ = current_pose_.pose.pose.position.y;

    current_pose_from_odom_ = current_pose_.pose.pose;
    current_velocity_ = current_pose_.twist.twist.linear.x;

    tf2::Quaternion q(
        current_pose_.pose.pose.orientation.x,
        current_pose_.pose.pose.orientation.y,
        current_pose_.pose.pose.orientation.z,
        current_pose_.pose.pose.orientation.w);

    // Extract yaw from the quaternion
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_yaw_ = yaw;
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

void StanleyControl::waypointCubeLinesCallback(const nav_msgs::msg::Path::SharedPtr msg){
    waypointsCubeLines.clear();
    for (const auto& pose : msg->poses) {
        Eigen::VectorXd waypoint(3);
        waypoint(0) = pose.pose.position.x;
        waypoint(1) = pose.pose.position.y;
        waypoint(2) = pose.pose.position.z;
        waypointsCubeLines.push_back(waypoint);
    }
}

double StanleyControl::getDistance(Eigen::VectorXd point1, Eigen::VectorXd point2){
    double distance = sqrt(pow(point1(0) - point2(0), 2) + pow(point1(1) - point2(1), 2));
    return distance;
}

void StanleyControl::setVelocityToSend(){
    if(closest_waypoint == 0){
        setVelocity = velocities[1];
    }else{
        // if(finishedpath){
        //     setVelocity = 0.0;
        // }else{
        //     setVelocity = velocities[closest_waypoint];
        // }
        setVelocity = velocities[closest_waypoint];
    }
    // RCLCPP_INFO(this->get_logger(), "setVelocity: %f", setVelocity);
}

void StanleyControl::waypointsComputation(){
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

    Eigen::VectorXd current_position(3);
    current_position << current_pose_from_odom_.position.x, current_pose_from_odom_.position.y, current_pose_from_odom_.position.z;

    double smallest_curr_distance = std::numeric_limits<double>::max();
    for (int i = search_start; i <= search_end; i++)
    {
        double curr_distance = getDistance(waypoints[i], current_position);
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

    double lookahead_actual = LookAheadDistance(current_velocity_); 


    // get the velocity of the current waypoint
    // double current_velocity = velocities[closest_waypoint];
    // RCLCPP_INFO(this->get_logger(), "current_velocity: %f", current_velocity);


    for (int i = closest_waypoint; i <= static_cast<int>(last_wp); i++)
    {
        double curr_distance = getDistance(waypoints[i], current_position);
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

    // RCLCPP_INFO(this->get_logger(), "lookahead_actual: %f", lookahead_actual);
    // RCLCPP_INFO(this->get_logger(), "target_waypoint: %d", target_waypoint);
}

void StanleyControl::marketTargetWaypoint(){
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

double StanleyControl::GetNormaliceAngle(double angle){
    if (angle > M_PI) angle -= 2 * M_PI;
    if (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

double StanleyControl::LookAheadDistance(double velocity){
    double clamped_speed = std::clamp(velocity, mps_alpha, mps_beta);
    return (lookahead_max - lookahead_min) / (mps_beta - mps_alpha) * (clamped_speed - mps_alpha) + lookahead_min;
}

// Stanely Controller
void StanleyControl::computeCrossTrackError(){
    double current_yaw_stanley = GetNormaliceAngle(current_yaw_);
    double fx = current_x_ + L * cos(current_yaw_stanley);
    double fy = current_y_ + L * sin(current_yaw_stanley);

    constexpr int search_offset_back = 5;
    constexpr int search_offset_forward = 15;
    constexpr size_t first_wp = 0;                        
    const size_t last_wp = waypoints.size() - 1; 

    int search_start = std::max(static_cast<int>(closest_waypoint) - search_offset_back, static_cast<int>(first_wp));
    int search_end = std::min(static_cast<int>(closest_waypoint) + search_offset_forward, static_cast<int>(last_wp));

    double min_dist = std::numeric_limits<double>::max();

    for (int i = search_start; i <= search_end; i++)
    {
        double dx = fx - waypoints[i](0);  // Access x-coordinate of Eigen::VectorXd
        double dy = fy - waypoints[i](1);  // Access y-coordinate of Eigen::VectorXd
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < min_dist) {
            min_dist = dist;
            target_idx = i;
        }
    }

    // Compute cross track error
    double front_axle_vec[2] = {-std::cos(current_yaw_stanley + M_PI / 2), -std::sin(current_yaw_stanley + M_PI / 2)};
    error_front_axle = (fx - waypoints[target_idx](0)) * front_axle_vec[0] + (fy - waypoints[target_idx](1)) * front_axle_vec[1];
}

void StanleyControl::computeSteeringAngle(){
    // double x_goal = waypoints[target_waypoint](0);
    // double y_goal = waypoints[target_waypoint](1);
    // double z_goal = waypoints[target_waypoint](2);
    double yaw_goal = waypoints[target_waypoint](3);


    // float alpha = atan2(y_goal, x_goal);
    // float lookahead_distance = sqrt(pow(x_goal, 2) + pow(y_goal, 2));
    // float purs_steering_angle = atan2(2.0 * wheelbase * sin(alpha), lookahead_distance);
    // float cross_track_error = error_front_axle * cross_track_err_rate;
    // float heading_error = yaw_goal * heading_err_rate;
    // stering_angle = pursuit_rate * purs_steering_angle + cross_track_error + heading_error;
    // RCLCPP_INFO(this->get_logger(), "stering_angle: %f", stering_angle);

    double current_yaw_car = GetNormaliceAngle(current_yaw_);
    double yaw_path = GetNormaliceAngle(yaw_goal);
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
