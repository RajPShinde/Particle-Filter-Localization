cmake_minimum_required(VERSION 3.2.1)
project (paticle_filter_localization)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  cv_bridge
  pcl_ros
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)


add_executable(particleFilter src/main.cpp src/particleFilter.cpp src/model.cpp include/particleFilter.hpp include/model.hpp)
target_link_libraries(particleFilter ${OpenCV_LIBS})
