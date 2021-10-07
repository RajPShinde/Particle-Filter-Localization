cmake_minimum_required(VERSION 3.2.1)
project (particle_filter_localization)

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
  nav_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
   INCLUDE_DIRS include
  )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)


add_executable(particleFilter src/main.cpp src/particleFilter.cpp src/model.cpp include/particleFilter.hpp include/model.hpp include/particle.hpp include/mapData.hpp)
target_link_libraries(particleFilter ${catkin_LIBRARIES})
