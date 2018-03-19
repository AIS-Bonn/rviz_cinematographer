cmake_minimum_required(VERSION 2.8.3)
project(laser_segmentation)

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  config_server
  nodelet
  tf
  eigen_conversions
  tf_conversions
  velodyne_pointcloud
  laser_geometry
  sensor_msgs
)

add_definitions(-DPCL_NO_PRECOMPILE=1)

catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ executable
add_executable(laser_segmentation_node
        src/laser_segmentation_node.cpp
        src/segmenter.cpp
)

## Add cmake target dependencies of the executable
add_dependencies(laser_segmentation_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(laser_segmentation_node
        ${catkin_LIBRARIES}
)

install(TARGETS laser_segmentation_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Declare a C++ library
add_library(laser_segmentation_nodelet
        src/laser_segmentation_nodelet.cpp
        src/segmenter.cpp
)

add_dependencies(laser_segmentation_nodelet
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(laser_segmentation_nodelet
        ${catkin_LIBRARIES}
)

install(TARGETS laser_segmentation_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)