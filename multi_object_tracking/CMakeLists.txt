cmake_minimum_required(VERSION 2.8)
project(multi_object_tracking)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DAPC_CAPTURE_DEBUG=1")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	nav_msgs
	std_srvs
	message_filters
	tf
	message_generation
	genmsg
	object_detection
	std_msgs
	geometry_msgs
)

add_definitions(-std=gnu++11 -fext-numeric-literals)

add_message_files(
	FILES
	DebugTracking.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS message_runtime
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIR}
)


add_library(hypotheses
	src/hypothesis.cpp
	src/hypothesis_3D.cpp
)

target_link_libraries(hypotheses
	${catkin_LIBRARIES}
)


add_executable(multi_object_tracking
	src/multi_object_tracker.cpp
	src/multihypothesistracker.cpp
	src/multiobjecttracker_algorithm.cpp
	src/hungarian.cpp
)

target_link_libraries(multi_object_tracking
	hypotheses
	${catkin_LIBRARIES}
)

## Add cmake target dependencies of the executable
add_dependencies(multi_object_tracking
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)

# Dummy target for IDE's
FILE(GLOB_RECURSE all_headers_for_ides
		${CMAKE_SOURCE_DIR}/**/*.hpp
		${CMAKE_SOURCE_DIR}/**/*.h)
ADD_CUSTOM_TARGET(ide_includes SOURCES ${all_headers_for_ides})