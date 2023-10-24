/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Based on Autoware waypoint saver, but here no Autoware message is used. 

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
#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <fstream>
#include <iomanip>

using namespace std;

// #include "CubicSpline1D.h"


class WaypointsLoader : public rclcpp::Node
{
private:
    /* data */
    std::string file_path_ = "/home/genis/Music/wp.csv";
    std::ofstream ofs_;
    vector<Eigen::VectorXd> waypoints;
    // vector<Eigen::VectorXd> waypointsCubeLines;

    void pub_callback();
    void ReadWaypoints();
    void visualizeNewWaypoints();
    geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);
    // void visualizeNewWaypointsCubeLines();


    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_loader_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_info_pub_;
    // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_loader_cubelines_pub_;
    // rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoint_loader_cubelines_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr speed_pub_; 


public:
    WaypointsLoader(/* args */);
    ~WaypointsLoader();
};

WaypointsLoader::WaypointsLoader(/* args */) : Node("waypoints_loader_node")
{
    timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&WaypointsLoader::pub_callback, this));
    waypoint_loader_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints_loaded", 10);
    waypoint_info_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints_info_loaded", 10);
    // waypoint_loader_cubelines_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoint_loader_cubelines", 10);
    // waypoint_loader_cubelines_pub_ = this->create_publisher<nav_msgs::msg::Path>("waypoint_loader_cubelines", 10);
    speed_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("waypointarray_speeds", 10);


    ReadWaypoints();

    // for (const auto& waypoint : waypoints)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Waypoint: [%f, %f, %f, %f, %f, %f]", 
    //         waypoint(0), waypoint(1), waypoint(2), waypoint(3), waypoint(4), waypoint(5));
    // }

    // vector<double> x, y;
    // for (const auto& waypoint : waypoints) {
    //     x.push_back(waypoint[0]);
    //     y.push_back(waypoint[1]);
    // }

    // std::vector<double> t_values(x.size());
    // std::iota(t_values.begin(), t_values.end(), 0);  
    // CubicSpline1D spline_x(t_values, x);
    // CubicSpline1D spline_y(t_values, y);

    // for (double t = 0; t < t_values.back(); t += 0.25) {
    //     double x_val = spline_x.calc_der0(t);
    //     double y_val = spline_y.calc_der0(t);

    //     Eigen::VectorXd point(2);
    //     point << x_val, y_val;
    //     waypointsCubeLines.push_back(point);
    // }

    RCLCPP_INFO(this->get_logger(), "waypoints_loader_node initialized");

}

WaypointsLoader::~WaypointsLoader()
{
}

void WaypointsLoader::pub_callback()
{
    visualizeNewWaypoints();
    // visualizeNewWaypointsCubeLines();
}


void WaypointsLoader::ReadWaypoints(){
    std::ifstream file(file_path_);
    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path_.c_str());
        return;
    }

    std::string line;
    // Skip the header line
    std::getline(file, line);

    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string value;
        Eigen::VectorXd waypoint(6); // x,y,z,yaw,velocity,change_flag

        for (int i = 0; i < 6; ++i)
        {
            std::getline(ss, value, ',');
            waypoint(i) = std::stod(value);
        }

        waypoints.push_back(waypoint);
    }

    file.close();
}


geometry_msgs::msg::Quaternion WaypointsLoader::yawToQuaternion(double yaw)
{

    tf2::Quaternion quaternion;
    
    quaternion.setRPY(0, 0, yaw);

    geometry_msgs::msg::Quaternion quaternion_msg;
    quaternion_msg.x = quaternion.x();
    quaternion_msg.y = quaternion.y();
    quaternion_msg.z = quaternion.z();
    quaternion_msg.w = quaternion.w();

    return quaternion_msg;
}

void WaypointsLoader::visualizeNewWaypoints() {
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::MarkerArray marker_array_info;
    std_msgs::msg::Float32MultiArray speed_array;


    for (size_t i = 0; i < waypoints.size(); ++i) {

        // yaw to quaternions conversion
        geometry_msgs::msg::Quaternion q = yawToQuaternion(waypoints[i](3));

        visualization_msgs::msg::Marker marker;
        marker.header.stamp = this->now();
        marker.header.frame_id = "odom";
        marker.id = i + 1000;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = waypoints[i](0);  // Assuming x is the first element
        marker.pose.position.y = waypoints[i](1); 
        marker.pose.position.z = waypoints[i](2);
        marker.pose.orientation = q;
        marker.scale.x = 0.5;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        
        marker_array.markers.push_back(marker);

        visualization_msgs::msg::Marker marker_info;
        marker_info.header.stamp = this->now();
        marker_info.header.frame_id = "odom";
        marker_info.id = i + 2000;
        marker_info.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker_info.action = visualization_msgs::msg::Marker::ADD;
        marker_info.pose.position.x = waypoints[i](0);  // Assuming x is the first element
        marker_info.pose.position.y = waypoints[i](1);
        marker_info.pose.position.z = waypoints[i](2);
        marker_info.pose.orientation = q;
        marker_info.scale.z = 0.4;
        marker_info.color.a = 1.0;
        marker_info.color.r = 1.0;
        marker_info.ns = "waypoints_info";
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << waypoints[i](4) << " km/h";
        marker_info.text = oss.str();

        marker_array_info.markers.push_back(marker_info);
        speed_array.data.push_back(waypoints[i](4)); 

    }
    
    waypoint_loader_pub_->publish(marker_array);
    waypoint_info_pub_->publish(marker_array_info);
    speed_pub_->publish(speed_array);
}

// void WaypointsLoader::visualizeNewWaypointsCubeLines(){
//     nav_msgs::msg::Path path;
//     path.header.stamp = this->now();
//     path.header.frame_id = "odom";

//     for (size_t i = 0; i < waypointsCubeLines.size(); ++i) {
//         geometry_msgs::msg::PoseStamped pose;
//         pose.header.stamp = this->now();
//         pose.header.frame_id = "odom";
//         pose.pose.position.x = waypointsCubeLines[i](0); 
//         pose.pose.position.y = waypointsCubeLines[i](1);
//         pose.pose.position.z = 0; 

//         path.poses.push_back(pose);
//     }

//     waypoint_loader_cubelines_pub_->publish(path);
// }

// void WaypointsLoader::visualizeNewWaypointsCubeLines(){
//     visualization_msgs::msg::MarkerArray marker_array;

//     for (size_t i = 0; i < waypointsCubeLines.size(); ++i) {

//         visualization_msgs::msg::Marker marker;
//         marker.header.stamp = this->now();
//         marker.header.frame_id = "odom";
//         marker.id = i + 3000;
//         marker.type = visualization_msgs::msg::Marker::SPHERE;
//         marker.action = visualization_msgs::msg::Marker::ADD;
//         marker.pose.position.x = waypointsCubeLines[i](0);  // Assuming x is the first element
//         marker.pose.position.y = waypointsCubeLines[i](1); 
//         marker.scale.x = 0.1;
//         marker.scale.y = 0.1;
//         marker.scale.z = 0.1;
//         marker.color.a = 1.0;
//         marker.color.r = 1.0;
//         marker.color.g = 0.0;
//         marker.color.b = 0.0;
        
//         marker_array.markers.push_back(marker);

//     }
    
//     waypoint_loader_cubelines_pub_->publish(marker_array);
// }


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointsLoader>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}