#include "segmentation.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "segmentation");
  ros::NodeHandle nh("~");
  segmentation::Segmentation Segmentation(nh);
  ros::spin();
  return 0;
}
