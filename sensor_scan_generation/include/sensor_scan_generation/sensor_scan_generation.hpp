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

#ifndef SENSOR_SCAN_GENERATION__SENSOR_SCAN_GENERATION_HPP_
#define SENSOR_SCAN_GENERATION__SENSOR_SCAN_GENERATION_HPP_

#include <memory>
#include <string>

#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace sensor_scan_generation
{

class SensorScanGenerationNode : public rclcpp::Node
{
public:
  explicit SensorScanGenerationNode(const rclcpp::NodeOptions & options);

private:
  void laserCloudAndOdometryHandler(
    const nav_msgs::msg::Odometry::ConstSharedPtr & odometry,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & laserCloud2);

  tf2::Transform getTransform(
    const std::string & target_frame, const std::string & source_frame, const rclcpp::Time & time);

  void publishTransform(
    const tf2::Transform & transform, const std::string & parent_frame,
    const std::string & child_frame, const rclcpp::Time & stamp);

  void publishOdometry(
    const tf2::Transform & transform, std::string parent_frame, const std::string & child_frame,
    const rclcpp::Time & stamp);

  std::string lidar_frame_;
  std::string base_frame_;
  std::string robot_base_frame_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_laser_cloud_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_chassis_odometry_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  message_filters::Subscriber<nav_msgs::msg::Odometry> odometry_sub_;
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> laser_cloud_sub_;

  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  tf2::Transform tf_lidar_to_robot_base_;
};

}  // namespace sensor_scan_generation

#endif  // SENSOR_SCAN_GENERATION__SENSOR_SCAN_GENERATION_HPP_
