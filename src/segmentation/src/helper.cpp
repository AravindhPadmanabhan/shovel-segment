#include "segmentation.h"

namespace segmentation {
void Segmentation::loadParameters() {
  if (nh_.getParam("topics/pub/segmented_pc", topic_segmented_pc_)) {
    ROS_INFO("Got param: %s", topic_segmented_pc_.c_str());
  } else {
    ROS_ERROR("Failed to get param 'topics/pub/segmented_pc'");
  }

  if (nh_.getParam("topics/sub/raw_pc", topic_raw_pc_)) {
    ROS_INFO("Got param: %s", topic_raw_pc_.c_str());
  } else {
    ROS_ERROR("Failed to get param 'topics/sub/raw_pc'");
  }

  if (nh_.getParam("frames/reference", reference_frame_)) {
    ROS_INFO("Got param: %s", reference_frame_.c_str());
  } else {
    ROS_ERROR("Failed to get param 'frames/reference'");
  }

  if (nh_.getParam("frames/source", source_frame_)) {
    ROS_INFO("Got param: %s", source_frame_.c_str());
  } else {
    ROS_ERROR("Failed to get param 'frames/source'");
  }

  if (nh_.getParam("distance_error_threhsold", err_threshold_)) {
    ROS_INFO("Got param: %f", err_threshold_);
  } else {
    ROS_ERROR("Failed to get param 'distance_error_threhsold'");
  }

  if (nh_.getParam("path_to_cad", path_to_cad_)) {
    ROS_INFO("Got param: %s", path_to_cad_.c_str());
  } else {
    ROS_ERROR("Failed to get param 'path_to_cad'");
  }

  std::ifstream input_file(path_to_cad_);
  if (input_file.is_open()) {
    if (igl::readSTL(input_file, V_, F_, N_)) {
      ROS_INFO("Loaded shovel CAD");
    } else {
      ROS_ERROR("Failed to load shovel CAD. Check file path.");
    }
  }
}

void Segmentation::advertiseTopics() {
  segmented_pc_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_segmented_pc_, 1);

}

void Segmentation::subscribeTopics() {
  raw_pc_subscriber_ = nh_.subscribe(topic_raw_pc_, 1, &Segmentation::segmentPC, this);
}

}  // namespace segmentation
