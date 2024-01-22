#include "segmentation.h"

namespace segmentation {
void Segmentation::loadParameters() {
  if (nh.getParam("topics/pub/segmented_pc", topic_segmented_pc_)) {
    ROS_INFO("Got param: %s", topic_segmented_pc_.c_str());
  } else {
    ROS_ERROR("Failed to get param 'topics/pub/segmented_pc'");
  }

  if (nh.getParam("topics/sub/raw_pc", topic_raw_pc_)) {
    ROS_INFO("Got param: %s", topic_raw_pc_.c_str());
  } else {
    ROS_ERROR("Failed to get param 'topics/sub/raw_pc'");
  }

  if (nh.getParam("distance_error_threhsold", err_threshold_)) {
    ROS_INFO("Got param: %f", err_threshold_);
  } else {
    ROS_ERROR("Failed to get param 'distance_error_threhsold'");
  }
}

void Segmentation::advertiseTopics() {
  segmented_pc_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_segmented_pc_, 1);

}

void Segmentation::subscribeTopics() {
  raw_pc_subscriber_.subscribe(nh_, topic_raw_pc_, 1);
}

}  // namespace segmentation
