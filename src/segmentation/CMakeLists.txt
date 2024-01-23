cmake_minimum_required(VERSION 3.18)
project(segmentation)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-O3)

# set(libigl_DIR "/home/aravindh/ETH/Semester Project/libigl/cmake")
include(FetchContent)
FetchContent_Declare(
  libigl
  GIT_REPOSITORY https://github.com/libigl/libigl.git
  GIT_TAG v2.4.0 # Replace with the desired version tag
)
FetchContent_MakeAvailable(libigl)

# find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.10.0 REQUIRED)
find_package(tf2_eigen)
# find_package(libigl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include CATKIN_DEPENDS message_runtime
)

# catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    # ${BOOST_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${LIBIGL_INCLUDE_DIR}
    include)

add_executable(${PROJECT_NAME} src/main.cpp
                               src/segmentation.cpp
                               src/helper.cpp
                               src/utils.cpp)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    # ${Boost_LIBRARIES}
    ${PCL_LIBRARIES}
    Eigen3::Eigen
    igl::core)

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)

