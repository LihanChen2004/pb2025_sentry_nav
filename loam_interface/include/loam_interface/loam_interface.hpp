// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef LOAM_INTERFACE__LOAM_INTERFACE_HPP_
#define LOAM_INTERFACE__LOAM_INTERFACE_HPP_

#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace loam_interface
{

class LoamInterfaceNode : public rclcpp::Node
{
public:
  explicit LoamInterfaceNode(const rclcpp::NodeOptions & options);

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

  void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcd_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  std::string state_estimation_topic_;
  std::string registered_scan_topic_;
  std::string odom_frame_;
  std::string lidar_frame_;
  std::string base_frame_;

  bool base_frame_to_lidar_initialized_;
  tf2::Transform tf_odom_to_lidar_odom_;
};

}  // namespace loam_interface

#endif  // LOAM_INTERFACE__LOAM_INTERFACE_HPP_
