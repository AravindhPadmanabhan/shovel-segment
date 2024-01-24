cmake_minimum_required(VERSION 3.18)
project(registration)  # Replace with your actual package name

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  tf
  tf_conversions
  # Add other ROS dependencies here
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs sensor_msgs pcl_ros tf tf_conversions
  # Add other catkin package dependencies here
)

catkin_install_python(PROGRAMS
  scripts/registration.py
  scripts/dataset.py 
  scripts/dataloader.py 
  scripts/tester.py 
  scripts/icp.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Uncomment if your package has include directories
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

# Uncomment if your package has configuration files
# install(DIRECTORY config
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# )
