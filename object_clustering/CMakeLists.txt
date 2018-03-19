cmake_minimum_required(VERSION 2.8)
project(object_clustering)

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

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")

add_definitions(-DPCL_NO_PRECOMPILE=1)


## Declare a C++ executable
add_executable(object_clustering_node
        src/object_clustering_node.cpp
        src/clusterer.cpp
)

## Add cmake target dependencies of the executable
add_dependencies(object_clustering_node
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(object_clustering_node
        ${catkin_LIBRARIES}
)

install(TARGETS object_clustering_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Declare a C++ library
add_library(object_clustering_nodelet
        src/object_clustering_nodelet.cpp
        src/clusterer.cpp
)

add_dependencies(object_clustering_nodelet
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(object_clustering_nodelet
        ${catkin_LIBRARIES}
)

install(TARGETS object_clustering_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
