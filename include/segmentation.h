#pragma once

#ifndef SEGMENTATION_H
#define SEGMENTATION_H

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <parameter_handler.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace segmentation {

class Segmentation {
 public:
  explicit Segmentation(const ros::NodeHandle &nh);

 private:
  ros::NodeHandle nh_;
  tf2_ros::TransformListener tfListener_;
  tf2_ros::Buffer tfBuffer_;

  // Topics
  std::string topic_segmented_pc_;
  std::string topic_raw_pc_;

  // Subscriber
  message_filters::Subscriber<sensor_msgs::PointCloud2> raw_pc_subscriber_;

  // Publishers
  ros::Publisher segmented_pc_publisher_;

  // Functions
  void loadParameters();
  void advertiseTopics();
  void subscribeTopics();

  // Parameters
  float err_threshold_;
};

}  // namespace segmentation

#endif  // SEGMENTATION_H
