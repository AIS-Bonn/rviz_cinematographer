cmake_minimum_required(VERSION 2.8)
project(dynamics_mapping)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	tf
	pcl_ros
	tf_conversions
	pcl_conversions
	eigen_conversions
	laser_segmentation
	config_server
	nodelet
)

catkin_package(
        INCLUDE_DIRS include
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

add_definitions(-DPCL_NO_PRECOMPILE=1)


## Declare a C++ executable
add_executable(${PROJECT_NAME}_node
        src/${PROJECT_NAME}_node.cpp
        src/dynamics_mapper.cpp
)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Declare a C++ library
add_library(${PROJECT_NAME}_nodelet
        src/${PROJECT_NAME}_nodelet.cpp
        src/dynamics_mapper.cpp
)

add_dependencies(${PROJECT_NAME}_nodelet
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_nodelet
        ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
