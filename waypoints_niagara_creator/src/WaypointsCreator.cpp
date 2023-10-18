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
// C++
#include <iostream>
#include <vector>
#include <fstream>
#include <iomanip>


using namespace std;

class WaypointsCreator : public rclcpp::Node
{
private:
  // variables 
  double current_pose_x_= 0;
  double current_pose_y_= 0;
  double current_pose_z_ = 0;

  double current_velocity_ = 0;


  double interval_ = 1.0;
  bool first_data_received_ = false;

  std::string file_path_ = "/home/genis/Music/wp.csv";
  std::ofstream ofs_;

  nav_msgs::msg::Odometry current_pose_;
  nav_msgs::msg::Odometry previous_pose_;

  // ROS2 functions
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  double computeDistance(const nav_msgs::msg::Odometry& odom_pos1 , const nav_msgs::msg::Odometry& odom_pos2);
  void writePoseToFile(const nav_msgs::msg::Odometry& odom_pos, bool change_flag);
  double yaw_callback(const nav_msgs::msg::Odometry& odom_pos) const;
  void waypointsPath();
  void writeHeaderToFile();
  void pub_callback();

  void displayMarker(const nav_msgs::msg::Odometry& odom_pos);
  void displayMarkerinfo(const nav_msgs::msg::Odometry& odom_pos);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_saver_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_info_pub_;


public:
  WaypointsCreator(/* args */);
  ~WaypointsCreator();
};

WaypointsCreator::WaypointsCreator(/* args */) : Node("waypoints_creator_node")
{
  odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&WaypointsCreator::odom_callback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&WaypointsCreator::pub_callback, this));
  waypoint_saver_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints", 10);
  waypoint_info_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoints_info", 10);


  writeHeaderToFile();
  
  RCLCPP_INFO(this->get_logger(), "waypoints_creator_node initialized");
}

WaypointsCreator::~WaypointsCreator()
{
  ofs_.close(); // Close the file
}

double WaypointsCreator::computeDistance(const nav_msgs::msg::Odometry& odom_pos1 , const nav_msgs::msg::Odometry& odom_pos2){
    double x1 = odom_pos1.pose.pose.position.x;
    double y1 = odom_pos1.pose.pose.position.y;
    double x2 = odom_pos2.pose.pose.position.x;
    double y2 = odom_pos2.pose.pose.position.y;
    return std::sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

void WaypointsCreator::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_pose_ = *msg;

  if (!first_data_received_) {
      first_data_received_ = true;
      RCLCPP_INFO(this->get_logger(), "First data received!");
      writePoseToFile(current_pose_, false);
      displayMarker(current_pose_);
      displayMarkerinfo(current_pose_);
      previous_pose_ = current_pose_;
  }

}


void WaypointsCreator::pub_callback()
{
  waypointsPath();
}

void WaypointsCreator::writeHeaderToFile() {
  ofs_.open(file_path_, std::ios::out);  // Open the file here
  if (!ofs_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open the file: %s", file_path_.c_str());
    return;
  }
  ofs_ << "x,y,z,yaw,velocity,change_flag" << std::endl;
}

void WaypointsCreator::writePoseToFile(const nav_msgs::msg::Odometry& odom_pos, bool change_flag){
  if (ofs_) {
    double current_pose_yaw_ = yaw_callback(odom_pos);
    ofs_ << std::fixed << std::setprecision(4) 
        << odom_pos.pose.pose.position.x << "," 
        << odom_pos.pose.pose.position.y << ","
        << odom_pos.pose.pose.position.z << "," 
        << current_pose_yaw_ << "," 
        << odom_pos.twist.twist.linear.x << ","
        << change_flag 
        << std::endl;
  }
  RCLCPP_INFO(this->get_logger(), "Enter to writePoseToFile");
}

double WaypointsCreator::yaw_callback(const nav_msgs::msg::Odometry& odom_pos) const {
    // Convert ROS2 Quaternion message to tf2 Quaternion
    tf2::Quaternion q(
        odom_pos.pose.pose.orientation.x,
        odom_pos.pose.pose.orientation.y,
        odom_pos.pose.pose.orientation.z,
        odom_pos.pose.pose.orientation.w);

    // Extract yaw from the quaternion
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
}

void WaypointsCreator::waypointsPath(){
  if (first_data_received_){
    double distance = computeDistance(current_pose_, previous_pose_);
      if (distance > interval_) {
      writePoseToFile(current_pose_, false);
      displayMarker(current_pose_);
      displayMarkerinfo(current_pose_);
      previous_pose_ = current_pose_;
      RCLCPP_INFO(this->get_logger(), "New waypoint added");
    }
  }
}


void WaypointsCreator::displayMarkerinfo(const nav_msgs::msg::Odometry& odom_pos){
  static int id = 3000;
  static visualization_msgs::msg::MarkerArray marker_array;
  geometry_msgs::msg::Pose current_pose_from_odom = odom_pos.pose.pose;

  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = this->now();
  marker.ns = "waypoints";
  marker.id = id;

  marker.scale.z = 0.4;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.ns = "waypoints_info";
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = current_pose_from_odom;

  std::ostringstream oss;
  oss << std::fixed << std::setprecision(2) << odom_pos.twist.twist.linear.x << " km/h";
  marker.text = oss.str();
  marker_array.markers.push_back(marker);
  waypoint_info_pub_->publish(marker_array);

  id++;

}


void WaypointsCreator::displayMarker(const nav_msgs::msg::Odometry& odom_pos){
  static int id = 0;
  static visualization_msgs::msg::MarkerArray marker_array;
  geometry_msgs::msg::Pose current_pose_from_odom = odom_pos.pose.pose;


  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "odom";
  marker.header.stamp = this->now();
  // marker.frame_locked = true;
  marker.ns = "waypoints";
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose = current_pose_from_odom;
  marker.scale.x = 0.5;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker_array.markers.push_back(marker);

  waypoint_saver_pub_->publish(marker_array);
  id++;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WaypointsCreator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
