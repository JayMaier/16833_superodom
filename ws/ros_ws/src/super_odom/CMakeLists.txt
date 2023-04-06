cmake_minimum_required(VERSION 3.0.2)
project(super_odom)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  cv_bridge
  image_transport
  random_numbers
  tf
  pcl_conversions
  pcl_ros
  gazebo_msgs
)


find_package(GTSAM REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAMCMakeTools REQUIRED)
set(BOOST_FIND_MINIMUM_VERSION 1.43)
set(BOOST_FIND_MINIMUM_COMPONENTS serialization system filesystem thread program_options date_time timer chrono regex )

##################
## ROS messages ##
##################
add_message_files(
  FILES
  CameraMeasurement.msg
  FeatureMeasurement.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  DEPENDS PCL
)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
    ${EIGEN_3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
    ${GTSAMCMakeTools_INCLUDE_DIRECTORIES}
)
 

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)

add_executable(vio_graph src/vio_graph.cpp src/parameters.cpp)
add_executable(imu_graph src/imu_graph.cpp)
target_link_libraries(vio_graph ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} gtsam gtsam_unstable)
target_link_libraries(imu_graph ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} gtsam gtsam_unstable)
