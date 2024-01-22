cmake_minimum_required(VERSION 3.0.0)
project(segmentation)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-O3)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED)
find_package(parameter_handler)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.10.0 REQUIRED)
find_package(libigl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include CATKIN_DEPENDS message_runtime
)

catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${BOOST_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${LIBIGL_INCLUDE_DIR}
    include)

cs_add_executable(${PROJECT_NAME} src/main.cpp
                                  src/segmentation.cpp
                                  src/helper.cpp
                                  src/utils.cpp)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    Eigen3::Eigen
    ${LIBIGL_LIBRARIES})

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)
cs_export()
