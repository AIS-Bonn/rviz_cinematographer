cmake_minimum_required(VERSION 2.8)
project(bhattacharyya_visualization)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	visualization_msgs
	config_server
	pcl_ros
	pcl_conversions
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories(include
	${catkin_INCLUDE_DIRS}
)

add_executable(bhattacharyya_visualization src/bhattacharyya_visualization.cpp)
target_link_libraries(bhattacharyya_visualization
	${PCL_LIBRARIES}
	${catkin_LIBRARIES}
)
