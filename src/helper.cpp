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

  if (nh.getParam("frames/reference", reference_frame_)) {
    ROS_INFO("Got param: %s", reference_frame_.c_str());
  } else {
    ROS_ERROR("Failed to get param 'frames/reference'");
  }

  if (nh.getParam("frames/source", source_frame_)) {
    ROS_INFO("Got param: %s", source_frame_.c_str());
  } else {
    ROS_ERROR("Failed to get param 'frames/source'");
  }

  if (nh.getParam("distance_error_threhsold", err_threshold_)) {
    ROS_INFO("Got param: %f", err_threshold_);
  } else {
    ROS_ERROR("Failed to get param 'distance_error_threhsold'");
  }
}

Eigen::Matrix4d Segmentation::getPoseIMU() {
  try {
    geometry_msgs::TransformStamped tf_stamped = tfBuffer_.lookupTransform("world", "shovel", ros::Time(0));
    auto tf_eigen = tf2::transformToEigen(tf_stamped);
    Eigen::Matrix4d tf_matrix = tf_eigen.matrix();
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    continue;
  }

  return tf_matrix;
}

void Segmentation::advertiseTopics() {
  segmented_pc_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_segmented_pc_, 1);

}

void Segmentation::subscribeTopics() {
  raw_pc_subscriber_ = nh_.subscribe(topic_raw_pc_, 1, &Segmentation::segmentPC, this);
}

}  // namespace segmentation
