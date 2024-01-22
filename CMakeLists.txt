cmake_minimum_required(VERSION 3.0.0)
project(segmentation)

set(CMAKE_CXX_STANDARD 14)
add_compile_options(-O3)

find_package(catkin_simple REQUIRED)
find_package(Boost REQUIRED)
find_package(parameter_handler)

catkin_simple(ALL_DEPS_REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${BOOST_INCLUDE_DIRS}
    include)

cs_add_executable(${PROJECT_NAME} src/main.cpp
                                  src/segmentation.cpp
                                  src/helper.cpp)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES})

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)
cs_export()
