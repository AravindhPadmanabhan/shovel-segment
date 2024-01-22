#include "segmentation.h"

#include <iostream>
#include <vector>

namespace segmentation {
Segmentation::Segmentation(const ros::NodeHandle &nh)
    : nh_(nh), tfListener_(tfBuffer_, nh) {
  loadParameters();
  advertiseTopics();
  subscribeTopics();
  ROS_INFO("Segmentation node is now running.");
}

void Segmentation::segmentPC(const sensor_msgs::PointCloud2::ConstPtr &pc_msg) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*pc_msg, cloud);

  // Convert PCL point cloud to Eigen matrix point cloud
  Eigen::MatrixXd cloud_matrix(cloud.points.size(), 3);
  for (size_t i = 0; i < cloud.points.size(); ++i) {
      cloud_matrix(i, 0) = cloud.points[i].x;
      cloud_matrix(i, 1) = cloud.points[i].y;
      cloud_matrix(i, 2) = cloud.points[i].z;
  }

  // Transform shovel mesh to IMU measured pose and segment points around it
  Eigen::MatrixXd shovel_imu_transform = Segmentation::getPoseIMU();
  Eigen::MatrixXd V_Transformed = Segmentation::transformPC(V_, shovel_imu_transform);
  Eigen::MatrixXd cloud_segmented_matrix = Segmentation::meshBasedSegmentation(V_Transformed, F_, cloud_matrix);

  // Convert segmented Eigen matrix points to PCL point cloud
  pcl::PointCloud<pcl::PointXYZ> cloud_segmented;
    for (int i = 0; i < cloud_segmented_matrix.rows(); ++i) {
        pcl::PointXYZ point;
        point.x = cloud_segmented_matrix(i, 0);
        point.y = cloud_segmented_matrix(i, 1);
        point.z = cloud_segmented_matrix(i, 2);
        cloud_segmented.push_back(point);
    }

  // Publish segmented points
  sensor_msgs::PointCloud2 segmented_pc_msg;
  pcl::toROSMsg(cloud_segmented, segmented_pc_msg)
  segmented_pc_msg.header.frame_id = reference_frame_; // Set the frame id as needed
  segmented_pc_msg.header.stamp = ros::Time::now();
  segmented_pc_publisher_.publish(segmented_pc_msg);

}

}  // namespace segmentation
