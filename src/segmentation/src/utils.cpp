#include "segmentation.h"
#include <igl/point_mesh_squared_distance.h>
#include <vector>

namespace segmentation {
Eigen::Matrix4d Segmentation::getPoseIMU() {
  Eigen::Matrix4d tf_matrix;
  try {
    geometry_msgs::TransformStamped tf_stamped = tfBuffer_.lookupTransform(reference_frame_, source_frame_, ros::Time(0));
    auto tf_eigen_element = tf2::transformToEigen(tf_stamped);
    tf_matrix = tf_eigen_element.matrix();
  } catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ros::Duration(1.0).sleep();
    // continue;
  }

  return tf_matrix;
}

Eigen::MatrixXd Segmentation::transformPC(const Eigen::MatrixXd& input_pc, const Eigen::MatrixXd& transform_matrix) {
  Eigen::MatrixXd homogenized_pc(input_pc.rows(), 4);
  homogenized_pc << input_pc, Eigen::MatrixXd::Ones(input_pc.rows(), 1);
  Eigen::MatrixXd transformed_pc_inter = homogenized_pc * transform_matrix.transpose();
  Eigen::MatrixXd transformed_pc = transformed_pc_inter.leftCols(3);

  return transformed_pc;
}

Eigen::MatrixXd Segmentation::meshBasedSegmentation(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, const Eigen::MatrixXd& pc) {
  Eigen::VectorXd sqrD; // list of signed distances
  Eigen::VectorXi I; // list of facet indices corresponding to smallest distances
  Eigen::MatrixXd C; // list of closest points // list of closest normals
  igl::point_mesh_squared_distance(pc,V,F,sqrD,I,C);

  std::vector<Eigen::Vector3d> segmented_points;
  for (int i = 0; i < C.rows(); i++) {
    if(sqrD[i] < err_threshold_) {
      segmented_points.push_back(C.row(i));
    }
  }

  Eigen::MatrixXd segmented_pc(segmented_points.size(), 3);
  for (int i = 0; i < segmented_points.size(); i++) {
    segmented_pc.row(i) = segmented_points[i];
  }

  return segmented_pc;
}

} // namespace segmentation

