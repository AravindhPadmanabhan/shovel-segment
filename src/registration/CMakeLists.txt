cmake_minimum_required(VERSION 3.0.2)
project(your_package_name)  # Replace with your actual package name

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
  scripts/your_script.py  # Replace with your actual script names
  # Add other Python scripts here
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
